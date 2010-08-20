/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/bitops.h>
#include <linux/mfd/pmic8058.h>

#include <mach/pmic8058-keypad.h>

#define PM8058_REV			0x002  /* PMIC4 revision */

#define PMIC8058_REV_A0			0xE1
#define PMIC8058_REV_B0			0xE2    /* REVISIT */

#define KEYP_CTRL			0x148

#define KEYP_CTRL_EVNTS			BIT(0)
#define KEYP_CTRL_EVNTS_MASK		0x3

#define KEYP_CTRL_SCAN_COLS_SHIFT	5
#define KEYP_CTRL_SCAN_COLS_MIN		5
#define KEYP_CTRL_SCAN_COLS_BITS	0x3

#define KEYP_CTRL_SCAN_ROWS_SHIFT	2
#define KEYP_CTRL_SCAN_ROWS_MIN		5
#define KEYP_CTRL_SCAN_ROWS_BITS	0x7

#define KEYP_CTRL_KEYP_EN		BIT(7)

#define KEYP_SCAN			0x149

#define KEYP_SCAN_READ_STATE		BIT(0)
#define KEYP_SCAN_DBOUNCE_SHIFT		1
#define KEYP_SCAN_PAUSE_SHIFT		3
#define KEYP_SCAN_ROW_HOLD_SHIFT	6

#define KEYP_TEST			0x14A

#define KEYP_TEST_CLEAR_RECENT_SCAN	BIT(6)
#define KEYP_TEST_CLEAR_OLD_SCAN	BIT(5)
#define KEYP_TEST_READ_RESET		BIT(4)
#define KEYP_TEST_DTEST_EN		BIT(3)
#define KEYP_TEST_ABORT_READ		BIT(0)

#define KEYP_TEST_DBG_SELECT_SHIFT	1

/* bits of these register represent
 * '0' for key press
 * '1' for key release
 */
#define KEYP_RECENT_DATA		0x14B
#define KEYP_OLD_DATA			0x14C

#define KEYP_CLOCK_FREQ			32768

/* Internal flags */
#define KEYF_FIX_LAST_ROW		0x01

/* ---------------------------------------------------------------------*/
struct pmic8058_kp {
	const struct pmic8058_keypad_data *pdata;
	struct input_dev *input;
	int key_sense_irq;
	int key_stuck_irq;

	unsigned short *keycodes;

	struct device *dev;
	u16 keystate[MATRIX_MAX_ROWS];

	u32	flags;
};

/* REVISIT - this should have come from pm8058_read/write */
static u8 rev = PMIC8058_REV_A0;

static int pmic8058_write_u8(u8 data, u16 reg)
{
	return pm8058_write(reg, &data, 1);
}

static int pmic8058_kp_write_u8(struct pmic8058_kp *kp,
				 u8 data, u16 reg)
{
	int rc;

	rc = pmic8058_write_u8(data, reg);
	if (rc < 0)
		dev_warn(kp->dev, "Error writing pmic8058: %X - ret %X\n",
				reg, rc);
	return rc;
}

static int pmic8058_kp_read(struct pmic8058_kp *kp,
				 u8 *data, u16 reg, unsigned num_bytes)
{
	int rc;

	rc = pm8058_read(reg, data, num_bytes);
	if (rc < 0)
		dev_warn(kp->dev, "Error reading pmic8058: %X - ret %X\n",
				reg, rc);

	return rc;
}

static u8 pmic8058_col_state(struct pmic8058_kp *kp, u8 col)
{
	/* all keys pressed on that particular row? */
	if (col == 0x00)
		return 1 << kp->pdata->num_cols;
	else
		return col & ((1 << kp->pdata->num_cols) - 1);
}
/* REVISIT: just for debugging, will be removed in final working version */
static void __dump_kp_regs(struct pmic8058_kp *kp, char *msg)
{
	u8 temp;

	dev_dbg(kp->dev, "%s\n", msg);

	pmic8058_kp_read(kp, &temp, KEYP_CTRL, 1);
	dev_dbg(kp->dev, "KEYP_CTRL - %X\n", temp);
	pmic8058_kp_read(kp, &temp, KEYP_SCAN, 1);
	dev_dbg(kp->dev, "KEYP_SCAN - %X\n", temp);
	pmic8058_kp_read(kp, &temp, KEYP_TEST, 1);
	dev_dbg(kp->dev, "KEYP_TEST - %X\n", temp);
}

/* H/W constraint:
 * One should read recent/old data registers equal to the
 * number of columns programmed in the keyp_control register,
 * otherwise h/w state machine may get stuck. In order to avoid this
 * situation one should check readstate bit in keypad scan
 * register to be '0' at the end of data read, to make sure
 * the keypad state machine is not in READ state.
 */
static int pmic8058_chk_read_state(struct pmic8058_kp *kp, u8 flag)
{
	u8 temp, scan_val;
	int retries = 10, rc;

	do {
		rc = pmic8058_kp_read(kp, &scan_val, KEYP_SCAN, 1);
		if (scan_val & 0x1) {
			if (flag)
				rc = pmic8058_kp_read(kp, &temp,
							 KEYP_RECENT_DATA, 1);
			else
				rc = pmic8058_kp_read(kp, &temp,
							 KEYP_OLD_DATA, 1);
		}
	} while ((scan_val & 0x1) && (--retries > 0));

	if (retries == 0)
		dev_dbg(kp->dev, "Unable to clear read state bit\n");

	return 0;
}

static int pmic8058_kp_read_matrix(struct pmic8058_kp *kp, u16 *new_state,
					 u16 *old_state)
{
	int rc, row, read_rows;
	u8 new_data[MATRIX_MAX_ROWS];
	u8 old_data[MATRIX_MAX_ROWS];

	if (kp->flags & KEYF_FIX_LAST_ROW)
		read_rows = MATRIX_MAX_ROWS;
	else
		read_rows = kp->pdata->num_rows;

	rc = pmic8058_kp_read(kp, new_data, KEYP_RECENT_DATA,
			      read_rows);

	if (!rc) {
		pmic8058_chk_read_state(kp, 1);
		for (row = 0; row < kp->pdata->num_rows; row++) {
			dev_dbg(kp->dev, "new_data[%d] = %d\n", row,
						new_data[row]);
			new_state[row] = pmic8058_col_state(kp, new_data[row]);
		}
	}

	if (old_state) {
		rc = pmic8058_kp_read(kp, old_data, KEYP_OLD_DATA,
				read_rows);
		if (!rc) {
			pmic8058_chk_read_state(kp, 0);
			for (row = 0; row < kp->pdata->num_rows; row++) {
				dev_dbg(kp->dev, "old_data[%d] = %d\n", row,
						 old_data[row]);
				old_state[row] = pmic8058_col_state(kp,
							 old_data[row]);
			}
		}
	}

	return rc;
}

static int __pmic8058_kp_scan_matrix(struct pmic8058_kp *kp, u16 *new_state,
					 u16 *old_state)
{
	int row, col, code;

	for (row = 0; row < kp->pdata->num_rows; row++) {
		int bits_changed = new_state[row] ^ old_state[row];

		if (!bits_changed)
			continue;

		for (col = 0; col < kp->pdata->num_cols; col++) {
			if (!(bits_changed & (1 << col)))
				continue;

			dev_dbg(kp->dev, "key [%d:%d] %s\n", row, col,
					!(new_state[row] & (1 << col)) ?
					"pressed" : "released");

			code = (row << 3) + col;
			input_event(kp->input, EV_MSC, MSC_SCAN, code);
			input_report_key(kp->input,
					kp->keycodes[code],
					!(new_state[row] & (1 << col)));

			input_sync(kp->input);
		}
	}

	return 0;
}

static int pmic8058_kp_scan_matrix(struct pmic8058_kp *kp, unsigned int events)
{
	u16 new_state[MATRIX_MAX_ROWS];
	u16 old_state[MATRIX_MAX_ROWS];
	int rc;

	dev_dbg(kp->dev, "events reported are %d\n", events);

	switch (events) {
	case 0x1:
		rc = pmic8058_kp_read_matrix(kp, new_state, NULL);
		__pmic8058_kp_scan_matrix(kp, new_state, kp->keystate);
		memcpy(kp->keystate, new_state, sizeof(new_state));
	break;
	case 0x3: /* two events - eventcounter is gray-coded */
		rc = pmic8058_kp_read_matrix(kp, new_state, old_state);
		__pmic8058_kp_scan_matrix(kp, old_state, kp->keystate);
		__pmic8058_kp_scan_matrix(kp, new_state, old_state);
		memcpy(kp->keystate, new_state, sizeof(new_state));
	break;
	case 0x2:
		dev_dbg(kp->dev, "Some key events are missed\n");
		rc = pmic8058_kp_read_matrix(kp, new_state, old_state);
		__pmic8058_kp_scan_matrix(kp, old_state, kp->keystate);
		__pmic8058_kp_scan_matrix(kp, new_state, old_state);
		memcpy(kp->keystate, new_state, sizeof(new_state));
	break;
	default:
		rc = -1;
	}
	return rc;
}

/* REVISIT */
static irqreturn_t pmic8058_kp_stuck_irq(int irq, void *data)
{
	struct pmic8058_kp *kp = data;

	dev_dbg(kp->dev, "some key got stuck\n");

	return IRQ_HANDLED;
}

static irqreturn_t pmic8058_kp_irq(int irq, void *data)
{
	struct pmic8058_kp *kp = data;
	u8 ctrl_val, events;
	int rc;

	dev_dbg(kp->dev, "key sense irq\n");
	__dump_kp_regs(kp, "pmic8058_kp_irq");

	rc = pmic8058_kp_read(kp, &ctrl_val, KEYP_CTRL, 1);
	events = ctrl_val & KEYP_CTRL_EVNTS_MASK;

	rc = pmic8058_kp_scan_matrix(kp, events);

	return IRQ_HANDLED;
}

static int pmic8058_kpd_init(struct pmic8058_kp *kp)
{
	int bits, rc, cycles;
	u8 scan_val = 0, ctrl_val = 0;
	static u8 row_bits[] = {
		0, 1, 2, 3, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7,
	};

	/* Find column bits */
	if (kp->pdata->num_cols < KEYP_CTRL_SCAN_COLS_MIN)
		bits = 0;
	else
		bits = kp->pdata->num_cols - KEYP_CTRL_SCAN_COLS_MIN;
	ctrl_val = (bits & KEYP_CTRL_SCAN_COLS_BITS) <<
		KEYP_CTRL_SCAN_COLS_SHIFT;

	/* Find row bits */
	if (kp->pdata->num_rows < KEYP_CTRL_SCAN_ROWS_MIN)
		bits = 0;
	else if (kp->pdata->num_rows > MATRIX_MAX_ROWS)
		bits = KEYP_CTRL_SCAN_ROWS_BITS;
	else
		bits = row_bits[kp->pdata->num_rows - KEYP_CTRL_SCAN_ROWS_MIN];

	/* Use max rows to fix last row problem if actual rows are less */
	if (kp->flags & KEYF_FIX_LAST_ROW)
		bits = KEYP_CTRL_SCAN_ROWS_BITS;
	ctrl_val |= (bits << KEYP_CTRL_SCAN_ROWS_SHIFT);

	rc = pmic8058_kp_write_u8(kp, ctrl_val, KEYP_CTRL);

	if (rev == PMIC8058_REV_A0)
		bits = fls(kp->pdata->debounce_ms) - 1;
	else
		bits = kp->pdata->debounce_ms / 5;

	scan_val |= (bits << KEYP_SCAN_DBOUNCE_SHIFT);

	bits = fls(kp->pdata->scan_delay_ms) - 1;
	scan_val |= (bits << KEYP_SCAN_PAUSE_SHIFT);

	/* Row hold time is a multiple of 32KHz cycles. */
	cycles = kp->pdata->row_hold_us * KEYP_CLOCK_FREQ / USEC_PER_SEC;
	bits = 3; /* Max : b11 */
	while (bits) {
		if (cycles >= (2 << bits))
			break;

		bits--;
	}
	scan_val |= (bits << KEYP_SCAN_ROW_HOLD_SHIFT);

	rc = pmic8058_kp_write_u8(kp, scan_val, KEYP_SCAN);

	return rc;
}

#ifdef CONFIG_PM
static int pmic8058_kp_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct pmic8058_kp *kp = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(kp->key_sense_irq);

	return 0;
}

static int pmic8058_kp_resume(struct platform_device *pdev)
{
	struct pmic8058_kp *kp = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(kp->key_sense_irq);

	return 0;
}
#else
#define pmic8058_kp_suspend	NULL
#define pmic8058_kp_resume	NULL
#endif

/*
 * keypad controller should be initialized in the following sequence
 * only, otherwise it might get into FSM stuck state.
 *
 * - Initialize keypad control parameters, like no. of rows, columns,
 *   timing values etc.,
 * - configure rows and column gpios pull up/down.
 * - set irq edge type.
 * - enable the keypad controller.
 */
static int __devinit pmic8058_kp_probe(struct platform_device *pdev)
{
	struct pmic8058_keypad_data *pdata = pdev->dev.platform_data;
	struct pmic8058_kp *kp;
	int rc, i;
	unsigned short *keycodes;
	u8 ctrl_val;

	if (!pdata || !pdata->num_cols || !pdata->num_rows ||
		pdata->num_cols > MATRIX_MAX_COLS ||
		pdata->num_rows > MATRIX_MAX_ROWS ||
		!pdata->keymap) {
		dev_err(&pdev->dev, "invalid platform data\n");
		return -EINVAL;
	}

	if (pdata->rows_gpio_start < 0 || pdata->cols_gpio_start < 0) {
		dev_err(&pdev->dev, "invalid gpio_start platform data\n");
		return -EINVAL;
	}

	if (!pdata->scan_delay_ms || pdata->scan_delay_ms > MAX_SCAN_DELAY
		|| pdata->scan_delay_ms < MIN_SCAN_DELAY ||
		!is_power_of_2(pdata->scan_delay_ms)) {
		dev_err(&pdev->dev, "invalid keypad scan time supplied\n");
		return -EINVAL;
	}

	rc = pm8058_read(PM8058_REV, &rev, 1);
	pr_info("PMIC4 is at %X revision\n", rev);

	if (rev == PMIC8058_REV_A0) {
		if (!pdata->debounce_ms || !is_power_of_2(pdata->debounce_ms)
				|| pdata->debounce_ms > MAX_DEBOUNCE_A0_TIME
				|| pdata->debounce_ms < MIN_DEBOUNCE_A0_TIME) {
			dev_err(&pdev->dev, "invalid debounce time supplied\n");
			return -EINVAL;
		}
	} else {
		if (!pdata->debounce_ms || ((pdata->debounce_ms % 5) != 0)
				|| pdata->debounce_ms > MAX_DEBOUNCE_B0_TIME
				|| pdata->debounce_ms < MIN_DEBOUNCE_B0_TIME) {
			dev_err(&pdev->dev, "invalid debounce time supplied\n");
			return -EINVAL;
		}
	}

	kp = kzalloc(sizeof(*kp), GFP_KERNEL);
	if (!kp)
		return -ENOMEM;

	keycodes = kzalloc(MATRIX_MAX_SIZE * sizeof(keycodes), GFP_KERNEL);
	if (!keycodes) {
		rc = -ENOMEM;
		goto err_alloc_mem;
	}

	platform_set_drvdata(pdev, kp);

	kp->pdata	= pdata;
	kp->dev		= &pdev->dev;
	kp->keycodes	= keycodes;

	/* REVISIT: actual revision with the fix */
	if (rev <= PMIC8058_REV_B0)
		kp->flags |= KEYF_FIX_LAST_ROW;

	kp->input = input_allocate_device();
	if (!kp->input) {
		dev_err(&pdev->dev, "unable to allocate input device\n");
		rc = -ENOMEM;
		goto err_alloc_device;
	}

	kp->key_sense_irq = platform_get_irq(pdev, 0);
	if (kp->key_sense_irq < 0) {
		dev_err(&pdev->dev, "unable to get keypad sense irq\n");
		rc = -ENXIO;
		goto err_get_irq;
	}

	kp->key_stuck_irq = platform_get_irq(pdev, 1);
	if (kp->key_stuck_irq < 0) {
		dev_err(&pdev->dev, "unable to get keypad stuck irq\n");
		rc = -ENXIO;
		goto err_get_irq;
	}

	if (pdata->input_name)
		kp->input->name = pdata->input_name;
	else
		kp->input->name = "PMIC8058 keypad";

	if (pdata->input_phys_device)
		kp->input->phys = pdata->input_phys_device;
	else
		kp->input->phys = "pmic8058_keypad/input0";

	kp->input->dev.parent	= &pdev->dev;

	kp->input->id.bustype	= BUS_HOST;
	kp->input->id.version	= 0x0001;
	kp->input->id.product	= 0x0001;
	kp->input->id.vendor	= 0x0001;

	kp->input->evbit[0]	= BIT_MASK(EV_KEY);

	if (pdata->rep)
		__set_bit(EV_REP, kp->input->evbit);

	kp->input->keycode	= keycodes;
	kp->input->keycodemax	= MATRIX_MAX_SIZE;
	kp->input->keycodesize	= sizeof(*keycodes);

	/* build keycodes for faster scanning */
	for (i = 0; i < pdata->keymap_size; i++) {
		unsigned int row = KEY_ROW(pdata->keymap[i]);
		unsigned int col = KEY_COL(pdata->keymap[i]);
		unsigned short keycode = KEY_VAL(pdata->keymap[i]);

		keycodes[(row << 3) + col] = keycode;
		__set_bit(keycode, kp->input->keybit);
	}
	__clear_bit(KEY_RESERVED, kp->input->keybit);

	input_set_capability(kp->input, EV_MSC, MSC_SCAN);
	input_set_drvdata(kp->input, kp);

	rc = input_register_device(kp->input);
	if (rc < 0) {
		dev_err(&pdev->dev, "unable to register keypad input device\n");
		goto err_get_irq;
	}

	/* initialize keypad state */
	memset(kp->keystate, 0xff, sizeof(kp->keystate));

	rc = pmic8058_kpd_init(kp);
	if (rc < 0) {
		dev_err(&pdev->dev, "unable to initialize keypad controller\n");
		goto err_kpd_init;
	}

	rc = pm8058_gpio_config_kypd_sns(pdata->cols_gpio_start,
						 pdata->num_cols);
	if (rc < 0) {
		dev_err(&pdev->dev, "unable to configure keypad sense lines\n");
		goto err_gpio_config;
	}

	rc = pm8058_gpio_config_kypd_drv(pdata->rows_gpio_start,
						 pdata->num_rows);
	if (rc < 0) {
		dev_err(&pdev->dev, "unable to configure keypad drive lines\n");
		goto err_gpio_config;
	}

	rc = request_irq(kp->key_sense_irq, pmic8058_kp_irq,
				 IRQF_TRIGGER_RISING, "pmic-keypad", kp);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to request keypad sense irq\n");
		goto err_req_sense_irq;
	}

	rc = request_irq(kp->key_stuck_irq, pmic8058_kp_stuck_irq,
				 IRQF_TRIGGER_RISING, "pmic-keypad-stuck", kp);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to request keypad stuck irq\n");
		goto err_req_stuck_irq;
	}

	rc = pmic8058_kp_read(kp, &ctrl_val, KEYP_CTRL, 1);
	ctrl_val |= KEYP_CTRL_KEYP_EN;
	rc = pmic8058_kp_write_u8(kp, ctrl_val, KEYP_CTRL);

	__dump_kp_regs(kp, "probe");

	device_init_wakeup(&pdev->dev, pdata->wakeup);

	return 0;

err_req_stuck_irq:
	free_irq(kp->key_sense_irq, NULL);
err_req_sense_irq:
err_gpio_config:
err_kpd_init:
	input_unregister_device(kp->input);
	kp->input = NULL;
err_get_irq:
	input_free_device(kp->input);
err_alloc_device:
	kfree(keycodes);
err_alloc_mem:
	kfree(kp);
	return rc;
}

static int __devexit pmic8058_kp_remove(struct platform_device *pdev)
{
	struct pmic8058_kp *kp = platform_get_drvdata(pdev);

	device_init_wakeup(&pdev->dev, 0);
	free_irq(kp->key_stuck_irq, NULL);
	free_irq(kp->key_sense_irq, NULL);
	input_unregister_device(kp->input);
	platform_set_drvdata(pdev, NULL);
	kfree(kp->input->keycode);
	kfree(kp);

	return 0;
}

static struct platform_driver pmic8058_kp_driver = {
	.probe		= pmic8058_kp_probe,
	.remove		= __devexit_p(pmic8058_kp_remove),
	.suspend	= pmic8058_kp_suspend,
	.resume		= pmic8058_kp_resume,
	.driver		= {
		.name = "pmic8058_keypad",
		.owner = THIS_MODULE,
	},
};

static int __init pmic8058_kp_init(void)
{
	return platform_driver_register(&pmic8058_kp_driver);
}
module_init(pmic8058_kp_init);

static void __exit pmic8058_kp_exit(void)
{
	platform_driver_unregister(&pmic8058_kp_driver);
}
module_exit(pmic8058_kp_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("PMIC8058 keypad driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pmic8058_keypad");

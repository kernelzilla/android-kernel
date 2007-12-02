/* linux/include/asm-arm/arch-msm/gpio.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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

#ifndef __ASM_ARCH_MSM_GPIO_H
#define __ASM_ARCH_MSM_GPIO_H

#include <linux/interrupt.h>

int gpio_request(unsigned gpio, const char *label);
void gpio_free(unsigned gpio);
int gpio_direction_input(unsigned gpio);
int gpio_direction_output(unsigned gpio, int value);
int gpio_get_value(unsigned gpio);
void gpio_set_value(unsigned gpio, int value);
int gpio_to_irq(unsigned gpio);

#include <asm-generic/gpio.h>

/* extended gpio api */

#define GPIOF_IRQF_MASK         0x0000ffff /* use to specify edge detection without */
#define GPIOF_IRQF_TRIGGER_NONE 0x00010000 /* IRQF_TRIGGER_NONE is 0 which also means "as already configured" */
#define GPIOF_INPUT             0x00020000
#define GPIOF_DRIVE_OUTPUT      0x00040000
#define GPIOF_OUTPUT_LOW        0x00080000
#define GPIOF_OUTPUT_HIGH       0x00100000

#define GPIOIRQF_SHARED         0x00000001 /* the irq line is shared with other inputs */

extern int gpio_configure(unsigned int gpio, unsigned long flags);
extern int gpio_read_detect_status(unsigned int gpio);
extern int gpio_clear_detect_status(unsigned int gpio);

#endif

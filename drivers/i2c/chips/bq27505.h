/* drivers/i2c/chips/bq27505.h
 *
 * Driver for Texas Instruments bq27505 battery fuel gauge
 *
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
 */

#ifndef BQ27505_H
#define BQ27505_H

enum bq27505_ioctl {
   BQ27505_ENTER_ROM_MODE,
   BQ27505_EXIT_ROM_MODE
};

#endif

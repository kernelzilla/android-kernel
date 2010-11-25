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

#include <linux/i2c.h>
#include <linux/mfd/tps65023.h>

/* TPS65023_registers */
#define TPS65023_VERSION	0
#define TPS65023_PGOODZ		1
#define TPS65023_MASK		2
#define TPS65023_REG_CTRL	3
#define TPS65023_CON_CTRL	4
#define TPS65023_CON_CTRL2	5
#define TPS65023_DEFCORE	6
#define TPS65023_DEFSLEW	7
#define TPS65023_LDO_CTRL	8
#define TPS65023_MAX		9

static struct i2c_client *tpsclient;

int tps65023_set_dcdc1_level(int mvolts)
{
	int val;
	int ret;

	if (!tpsclient)
		return -ENODEV;

	if (mvolts < 800 || mvolts > 1600)
		return -EINVAL;

	if (mvolts == 1600)
		val = 0x1F;
	else
		val = ((mvolts - 800)/25) & 0x1F;

	ret = i2c_smbus_write_byte_data(tpsclient, TPS65023_DEFCORE, val);

	if (!ret)
		ret = i2c_smbus_write_byte_data(tpsclient,
				TPS65023_CON_CTRL2, 0x80);

	return ret;
}
EXPORT_SYMBOL(tps65023_set_dcdc1_level);

int tps65023_get_dcdc1_level(int *mvolts)
{
	int val;

	if (!tpsclient)
		return -ENODEV;

	val = i2c_smbus_read_byte_data(tpsclient, TPS65023_DEFCORE) & 0x1F;

	if (val == 0x1F)
		*mvolts = 1600;
	else
		*mvolts = (val * 25) + 800;
	return 0;
}
EXPORT_SYMBOL(tps65023_get_dcdc1_level);

static int tps65023_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR "TPS65023 does not support SMBUS_BYTE_DATA.\n");
		return -EINVAL;
	}

	tpsclient = client;
	printk(KERN_INFO "TPS65023: PMIC probed.\n");
	return 0;
}

static int __devexit tps65023_remove(struct i2c_client *client)
{
	tpsclient = NULL;
	return 0;
}

static const struct i2c_device_id tps65023_id[] = {
	{ "tps65023", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tps65023_id);

static struct i2c_driver tps65023_driver = {
	.driver = {
		.name   = "tps65023",
		.owner  = THIS_MODULE,
	},
	.probe  = tps65023_probe,
	.remove = __devexit_p(tps65023_remove),
	.id_table = tps65023_id,
};

static int __init tps65023_init(void)
{
	return i2c_add_driver(&tps65023_driver);
}


static void __exit tps65023_exit(void)
{
	i2c_del_driver(&tps65023_driver);
}

module_init(tps65023_init);
module_exit(tps65023_exit);

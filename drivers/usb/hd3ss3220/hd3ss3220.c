/*
 * Copyright (C) 2018, Paul Keith <javelinanddart@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <fih/hwid.h>

#define HD3SS3220_DRIVER_NAME "hd3ss3220"
#define COMPATIBLE_NAME "ti,hd3ss3220"

#define HD3SS3220_DEVICE_I2C_ADDR_LOW	0x60
#define HD3SS3220_DEVICE_I2C_ADDR_HIGH	0x61

#define NEW_HD3SS3220_DEVICE_I2C_ADDR_LOW	0x47
#define NEW_HD3SS3220_DEVICE_I2C_ADDR_HIGH	0x67

#define I2C_RETRY_MAX 10

static const struct of_device_id hd3ss3220_match_table[] = {
	{ .compatible = COMPATIBLE_NAME },
	{ }
};

static struct i2c_device_id hd3ss3220_i2c_id1[] = {
	{ HD3SS3220_DRIVER_NAME, HD3SS3220_DEVICE_I2C_ADDR_LOW },
	{ }
};

static struct i2c_device_id hd3ss3220_i2c_id2[] = {
	{ HD3SS3220_DRIVER_NAME, HD3SS3220_DEVICE_I2C_ADDR_HIGH },
	{ }
};

static struct i2c_device_id hd3ss3220_i2c_id3[] = {
	{ HD3SS3220_DRIVER_NAME, NEW_HD3SS3220_DEVICE_I2C_ADDR_HIGH },
	{ }
};

static int hd3ss3220_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct regulator *regulator_vdd;
	struct i2c_msg msg;
	u8 buffer[2];
	int i, ret = 0;

	// Enable the regulators
	regulator_vdd = regulator_get(&client->dev," hd3ss3220vdd");
	if (IS_ERR(regulator_vdd)) {
		pr_err("%s: Failed to get hd3ss3220vdd\n", __func__);
		ret = PTR_ERR(regulator_vdd);
		regulator_put(regulator_vdd);
		return ret;
	}
	ret = regulator_enable(regulator_vdd);
	if (ret) {
		pr_err("%s: Failed to enable vdd-supply\n", __func__);
		regulator_disable(regulator_vdd);
		regulator_put(regulator_vdd);
		return ret;
	}

	regulator_vdd = regulator_get(&client->dev, "usb_redriver");
		if (IS_ERR(regulator_vdd)) {
		pr_err("%s: Failed to get usb_redriver\n", __func__);
		ret = PTR_ERR(regulator_vdd);
		regulator_put(regulator_vdd);
		return ret;
	}
	ret = regulator_enable(regulator_vdd);
	if (ret) {
		pr_err("%s: Failed to enable vdd-supply\n", __func__);
		regulator_disable(regulator_vdd);
		regulator_put(regulator_vdd);
		return ret;
	}

	// Write UFP mode to the registers
	buffer[0] = 0x0A;
	buffer[1] = 0x10;
	msg.flags = 0;
	msg.addr = id->driver_data;
	msg.buf = buffer;
	msg.len = sizeof(buffer);

	for (i = 0; i < I2C_RETRY_MAX; i++) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret != 1) {
			pr_err("%s: Retrying I2C write of UFP mode %d\n", __func__, i);
			ret = -EIO;
			msleep(20);
		} else {
			ret = 0;
			break;
		}
	}

	return ret;
}

static int hd3ss3220_i2c_remove(struct i2c_client *client)
{
	struct regulator *regulator_vdd;

	// Disable the regulators
	regulator_vdd = regulator_get(&client->dev," hd3ss3220vdd");
	if (IS_ERR(regulator_vdd)) {
		pr_err("%s: Failed to get hd3ss3220vdd\n", __func__);
		regulator_put(regulator_vdd);
	} else {
		regulator_disable(regulator_vdd);
		regulator_put(regulator_vdd);
	}

	regulator_vdd = regulator_get(&client->dev, "usb_redriver");
	if (IS_ERR(regulator_vdd)) {
		pr_err("%s: Failed to get usb_redriver\n", __func__);
		regulator_put(regulator_vdd);
	} else {
		regulator_disable(regulator_vdd);
		regulator_put(regulator_vdd);
	}

	return 0;
}

static struct i2c_driver hd3ss3220_i2c_driver = {
	.driver = {
		.name = HD3SS3220_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = hd3ss3220_match_table,
	},
	.probe = hd3ss3220_i2c_probe,
	.remove =  hd3ss3220_i2c_remove,
	.id_table = hd3ss3220_i2c_id3,
};

static void correct_i2c_driver_id_table(void)
{
	int project = fih_hwid_fetch(FIH_HWID_PRJ);
	int revision = fih_hwid_fetch(FIH_HWID_REV);

	if (project == FIH_PRJ_NBQ) {
		if (revision <= FIH_REV_EVT_PRE1_5) {
			hd3ss3220_i2c_driver.id_table = hd3ss3220_i2c_id1;
		} else if (revision == FIH_REV_EVT1C) {
			hd3ss3220_i2c_driver.id_table = hd3ss3220_i2c_id2;
		}
	}
}

static int __init hd3ss3220_init(void)
{
	correct_i2c_driver_id_table();
	return i2c_add_driver(&hd3ss3220_i2c_driver);
}

static void __exit hd3ss3220_exit(void)
{
	i2c_del_driver(&hd3ss3220_i2c_driver);
}

module_init(hd3ss3220_init);
module_exit(hd3ss3220_exit);

MODULE_DESCRIPTION("Basic TI HD3SS3220 UFP driver");
MODULE_AUTHOR("Paul Keith <javelinanddart@gmail.com>");
MODULE_LICENSE("GPL");

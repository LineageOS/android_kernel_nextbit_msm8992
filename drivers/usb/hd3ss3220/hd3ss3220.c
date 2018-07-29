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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <fih/hwid.h>

#define HD3SS3220_DRIVER_NAME "hd3ss3220"
#define COMPATIBLE_NAME "ti,hd3ss3220"

#define HD3SS3220_DEVICE_I2C_ADDR_LOW	0x60
#define HD3SS3220_DEVICE_I2C_ADDR_HIGH	0x61

#define NEW_HD3SS3220_DEVICE_I2C_ADDR_LOW	0x47
#define NEW_HD3SS3220_DEVICE_I2C_ADDR_HIGH	0x67

#define I2C_RETRY_MAX 10

#define MODE_UFP 0x10
#define MODE_DFP 0x20

struct hd3ss3220_client_data {
	struct i2c_adapter *i2c_bus;
	struct regulator *hd3ss3220vdd;
	struct regulator *usb_redriver;
	struct work_struct hd3ss3220_mode_work;
	unsigned addr;
	unsigned gpio;
	unsigned irq;
};

static void write_hd3ss3220_mode(struct work_struct *work)
{
	struct hd3ss3220_client_data *data = container_of(work,
			struct hd3ss3220_client_data, hd3ss3220_mode_work);
	struct i2c_msg msg_read[2];
	struct i2c_msg msg_write;
	u8 msg_buf_read[2];
	u8 msg_buf_write[2];
	int i, ret = 0;

	/* Setup the read msg */
	msg_buf_read[0] = 0x0A;
	msg_read[0].flags = 0;
	msg_read[0].addr = data->addr;
	msg_read[0].buf = &msg_buf_read[0];
	msg_read[0].len = sizeof(msg_buf_read[0]);
	msg_read[1].flags = I2C_M_RD;
	msg_read[1].addr = data->addr;
	msg_read[1].buf = &msg_buf_read[1];
	msg_read[1].len = sizeof(msg_buf_read[1]);

	/* Read the control register */
	for (i = 0; i < I2C_RETRY_MAX; i++) {
		ret = i2c_transfer(data->i2c_bus, msg_read, 2);
		if (ret == 2)
			break;
		pr_err("%s: Retrying I2C read of control register %d\n",
				__func__, i);
		msleep(20);
	}

	/* Setup the write msg */
	msg_buf_write[0] = 0x0A;
	msg_write.flags = 0;
	msg_write.addr = data->addr;
	msg_write.buf = msg_buf_write;
	msg_write.len = sizeof(msg_buf_write);

	/* Clear the old mode, and set the new one */
	msg_buf_write[1] = msg_buf_read[1];
	msg_buf_write[1] &= ~(MODE_UFP | MODE_DFP);
	msg_buf_write[1] |= MODE_UFP;

	/* Write the control register */
	for (i = 0; i < I2C_RETRY_MAX; i++) {
		ret = i2c_transfer(data->i2c_bus, &msg_write, 1);
		if (ret == 1)
			break;
		pr_err("%s: Retrying I2C write of control register %d\n",
				__func__, i);
		msleep(20);
	}
}

static irqreturn_t hd3ss3220_irq_handler(int irq, void *data)
{
	/* Write UFP mode to device */
	struct i2c_client *client = to_i2c_client(data);
	struct hd3ss3220_client_data *client_data = i2c_get_clientdata(client);

	cancel_work_sync(&client_data->hd3ss3220_mode_work);
	schedule_work(&client_data->hd3ss3220_mode_work);
	return IRQ_HANDLED;
}

static int hd3ss3220_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct hd3ss3220_client_data *data;
	int ret = 0;

	data = kzalloc(sizeof(struct hd3ss3220_client_data), GFP_KERNEL);

	/* Enable the regulators */
	data->hd3ss3220vdd = regulator_get(&client->dev, "hd3ss3220vdd");
	if (IS_ERR(data->hd3ss3220vdd)) {
		pr_err("%s: Failed to get hd3ss3220vdd\n", __func__);
		ret = PTR_ERR(data->hd3ss3220vdd);
		goto err_hd3ss3220vdd_reg_get;
	}
	ret = regulator_enable(data->hd3ss3220vdd);
	if (ret) {
		pr_err("%s: Failed to enable vdd-supply\n", __func__);
		goto err_hd3ss3220vdd_reg_en;
	}

	data->usb_redriver = regulator_get(&client->dev, "usb_redriver");
	if (IS_ERR(data->usb_redriver)) {
		pr_err("%s: Failed to get usb_redriver\n", __func__);
		ret = PTR_ERR(data->usb_redriver);
		goto err_redriver_reg_get;
	}
	ret = regulator_enable(data->usb_redriver);
	if (ret) {
		pr_err("%s: Failed to enable vdd-supply\n", __func__);
		goto err_redriver_reg_en;
	}

	/* Setup I2C messages for writing device mode */
	data->addr = id->driver_data;
	data->i2c_bus = client->adapter;

	/* Write UFP mode to device */
	INIT_WORK(&data->hd3ss3220_mode_work, write_hd3ss3220_mode);
	schedule_work(&data->hd3ss3220_mode_work);

	/* Setup IRQ handling */
	ret = of_get_named_gpio(client->dev.of_node,
			"ti-hd3ss3220,irq-gpio", 0);
	if (ret < 0) {
		pr_err("%s: Failed to get IRQ gpio\n", __func__);
		goto err_gpio_failed;
	}

	data->gpio = ret;

	ret = gpio_to_irq(data->gpio);
	if (ret < 0) {
		pr_err("%s: Failed to get IRQ from gpio\n", __func__);
		goto err_irq_failed;
	}

	data->irq = ret;

	ret = request_threaded_irq(data->irq, NULL,
			hd3ss3220_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			HD3SS3220_DRIVER_NAME, &client->dev);
	if (ret) {
		pr_err("%s: Failed to get IRQ handler\n", __func__);
		goto err_irq_failed;
	}

	i2c_set_clientdata(client, data);

	return ret;

err_irq_failed:
	gpio_free(data->gpio);
err_gpio_failed:
	cancel_work_sync(&data->hd3ss3220_mode_work);
	regulator_disable(data->usb_redriver);
err_redriver_reg_en:
	regulator_put(data->usb_redriver);
err_redriver_reg_get:
	regulator_disable(data->hd3ss3220vdd);
err_hd3ss3220vdd_reg_en:
	regulator_put(data->hd3ss3220vdd);
err_hd3ss3220vdd_reg_get:
	kfree(data);
	return ret;
}

static int hd3ss3220_i2c_remove(struct i2c_client *client)
{
	struct hd3ss3220_client_data *data = i2c_get_clientdata(client);

	if (data) {
		cancel_work_sync(&data->hd3ss3220_mode_work);
		free_irq(data->irq, &client->dev);
		gpio_free(data->gpio);
		regulator_disable(data->usb_redriver);
		regulator_put(data->usb_redriver);
		regulator_disable(data->hd3ss3220vdd);
		regulator_put(data->hd3ss3220vdd);
		kfree(data);
	}

	return 0;
}

static const struct of_device_id hd3ss3220_match_table[] = {
	{ .compatible = COMPATIBLE_NAME },
	{ }
};

static const struct i2c_device_id hd3ss3220_i2c_id1[] = {
	{ HD3SS3220_DRIVER_NAME, HD3SS3220_DEVICE_I2C_ADDR_LOW },
	{ }
};
static const struct i2c_device_id hd3ss3220_i2c_id2[] = {
	{ HD3SS3220_DRIVER_NAME, HD3SS3220_DEVICE_I2C_ADDR_HIGH },
	{ }
};

static const struct i2c_device_id hd3ss3220_i2c_id3[] = {
	{ HD3SS3220_DRIVER_NAME, NEW_HD3SS3220_DEVICE_I2C_ADDR_HIGH },
	{ }
};

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

static void correct_i2c_device_address(struct i2c_driver *driver)
{
	int project = fih_hwid_fetch(FIH_HWID_PRJ);
	int revision = fih_hwid_fetch(FIH_HWID_REV);

	if (project == FIH_PRJ_NBQ) {
		if (revision <= FIH_REV_EVT_PRE1_5)
			driver->id_table = hd3ss3220_i2c_id1;
		else if (revision == FIH_REV_EVT1C)
			driver->id_table = hd3ss3220_i2c_id2;
	}
}

static int __init hd3ss3220_init(void)
{
	correct_i2c_device_address(&hd3ss3220_i2c_driver);
	return i2c_add_driver(&hd3ss3220_i2c_driver);
}

static void __exit hd3ss3220_exit(void)
{
	i2c_del_driver(&hd3ss3220_i2c_driver);
}

module_init(hd3ss3220_init);
module_exit(hd3ss3220_exit);

MODULE_DESCRIPTION("Simple TI HD3SS3220 UFP driver");
MODULE_AUTHOR("Paul Keith <javelinanddart@gmail.com>");
MODULE_LICENSE("GPL");

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
	struct i2c_msg msg;
	u8 msg_buf[2];
	int i, ret = 0;

	/* Setup the read msg */
	msg_buf[0] = 0x0A;
	msg.flags = I2C_M_RD;
	msg.addr = data->addr;
	msg.buf = msg_buf;
	msg.len = sizeof(msg_buf);

	/* Read the control register */
	for (i = 0; i < I2C_RETRY_MAX; i++) {
		ret = i2c_transfer(data->i2c_bus, &msg, 1);
		if (ret == 1)
			break;
		pr_err("%s: Retrying I2C read at addr=0x%x of register=0x%x err=%d try=%d\n",
				__func__, data->addr, msg_buf[0], ret, i);
		msleep(20);
	}

	/* Setup the write msg */
	msg.flags = 0;

	/* Clear the old mode, and set the new one */
	msg_buf[1] &= ~(MODE_UFP | MODE_DFP);
	msg_buf[1] |= MODE_UFP;

	/* Write the control register */
	for (i = 0; i < I2C_RETRY_MAX; i++) {
		ret = i2c_transfer(data->i2c_bus, &msg, 1);
		if (ret == 1)
			break;
		pr_err("%s: Retrying I2C write at addr=0x%x of register=0x%x value=0x%x err=%d try=%d\n",
				__func__, data->addr, msg_buf[0], msg_buf[1],
				ret, i);
		msleep(20);
	}
}

static irqreturn_t hd3ss3220_irq_handler(int irq, void *data)
{
	/* Schedule write of UFP mode to device */
	struct i2c_client *client = to_i2c_client(data);
	struct hd3ss3220_client_data *client_data = i2c_get_clientdata(client);

	schedule_work(&client_data->hd3ss3220_mode_work);

	return IRQ_HANDLED;
}

static int hd3ss3220_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct hd3ss3220_client_data *data;
	int ret = 0;

	data = kzalloc(sizeof(struct hd3ss3220_client_data), GFP_KERNEL);

	if (!data) {
		ret = -ENOMEM;
		goto done;
	}

	/* Enable the regulators */
	data->hd3ss3220vdd = regulator_get(&client->dev, "hd3ss3220vdd");
	if (IS_ERR(data->hd3ss3220vdd)) {
		ret = PTR_ERR(data->hd3ss3220vdd);
		pr_err("%s: Failed to get hd3ss3220vdd supply err=%d\n",
				__func__, ret);
		goto err_hd3ss3220vdd_reg_get;
	}
	ret = regulator_enable(data->hd3ss3220vdd);
	if (ret) {
		pr_err("%s: Failed to enable hd3ss3220vdd supply err=%d\n",
				__func__, ret);
		goto err_hd3ss3220vdd_reg_en;
	}

	data->usb_redriver = regulator_get(&client->dev, "usb_redriver");
	if (IS_ERR(data->usb_redriver)) {
		ret = PTR_ERR(data->usb_redriver);
		pr_err("%s: Failed to get usb_redriver supply err=%d\n",
				__func__, ret);
		goto err_redriver_reg_get;
	}
	ret = regulator_enable(data->usb_redriver);
	if (ret) {
		pr_err("%s: Failed to enable usb_redriver supply err=%d\n",
				__func__, ret);
		goto err_redriver_reg_en;
	}

	/* Setup I2C messages for writing device mode */
	data->addr = id->driver_data;
	data->i2c_bus = client->adapter;

	/* Schedule write of UFP mode to device */
	INIT_WORK(&data->hd3ss3220_mode_work, write_hd3ss3220_mode);
	schedule_work(&data->hd3ss3220_mode_work);

	/* Setup IRQ handling */
	ret = of_get_named_gpio(client->dev.of_node,
			"ti-hd3ss3220,irq-gpio", 0);
	if (ret < 0) {
		pr_err("%s: Failed to get IRQ gpio err=%d\n", __func__, ret);
		goto err_gpio_failed;
	}

	data->gpio = ret;

	ret = gpio_to_irq(data->gpio);
	if (ret < 0) {
		pr_err("%s: Failed to get IRQ from gpio err=%d\n", __func__,
				ret);
		goto err_irq_failed;
	}

	data->irq = ret;

	ret = request_threaded_irq(data->irq, NULL,
			hd3ss3220_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			HD3SS3220_DRIVER_NAME, &client->dev);
	if (ret) {
		pr_err("%s: Failed to create IRQ handler err=%d\n", __func__,
				ret);
		goto err_irq_failed;
	}

	i2c_set_clientdata(client, data);

	goto done;

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
done:
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

static struct i2c_device_id hd3ss3220_i2c_id[] = {
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
	.id_table = hd3ss3220_i2c_id,
};

static void correct_i2c_device_address(struct i2c_device_id *id)
{
	int project = fih_hwid_fetch(FIH_HWID_PRJ);
	int revision = fih_hwid_fetch(FIH_HWID_REV);

	if (project == FIH_PRJ_NBQ) {
		if (revision <= FIH_REV_EVT_PRE1_5)
			id->driver_data = HD3SS3220_DEVICE_I2C_ADDR_LOW;
		else if (revision == FIH_REV_EVT1C)
			id->driver_data = HD3SS3220_DEVICE_I2C_ADDR_HIGH;
	}
}

static int __init hd3ss3220_init(void)
{
	correct_i2c_device_address(hd3ss3220_i2c_id);
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

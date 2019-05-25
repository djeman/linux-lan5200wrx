/*
 * Driver for MAXIM MAX11803 - A Resistive touch screen controller with
 * i2c interface
 *
 * Based on max11801_ts.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/delay.h>

/* Register Address define */
#define GENERAL_STATUS_REG		0x00
#define GENERAL_CONF_REG		0x01
#define MESURE_RES_CONF_REG		0x02
#define MESURE_AVER_CONF_REG		0x03
#define ADC_SAMPLE_TIME_CONF_REG	0x04
#define PANEL_SETUP_TIME_CONF_REG	0x05
#define DELAY_CONVERSION_CONF_REG	0x06
#define TOUCH_DETECT_PULLUP_CONF_REG	0x07
#define AUTO_MODE_TIME_CONF_REG		0x08
#define APERTURE_CONF_REG		0x09
#define AUX_MESURE_CONF_REG		0x0a
#define OP_MODE_CONF_REG		0x0b

#define XY_BUFSIZE			4
#define XY_BUF_OFFSET			4

#define MAX11803_MAX_X			0xfff
#define MAX11803_MAX_Y			0xfff

#define MEASURE_TAG_OFFSET		2
#define MEASURE_TAG_MASK		(3 << MEASURE_TAG_OFFSET)
#define EVENT_TAG_OFFSET		0
#define EVENT_TAG_MASK			(3 << EVENT_TAG_OFFSET)
#define MEASURE_X_TAG			(0 << MEASURE_TAG_OFFSET)
#define MEASURE_Y_TAG			(1 << MEASURE_TAG_OFFSET)

#define MAX11803_EDGE_INT		(1 << 1)
#define MAX11803_CONT_INT		(1 << 0)

#define Panel_Setup_X			(0x69 << 1)
#define Panel_Setup_Y			(0x6b << 1)

#define XY_combined_measurement		(0x70 << 1)
#define X_measurement			(0x78 << 1)
#define Y_measurement			(0x7a << 1)
#define AUX_measurement			(0x76 << 1)

#define FIFO_RD_X_MSB			(0x52 << 1)
#define FIFO_RD_X_LSB			(0x53 << 1)
#define FIFO_RD_Y_MSB			(0x54 << 1)
#define FIFO_RD_Y_LSB			(0x55 << 1)
#define FIFO_RD_AUX_MSB			(0x5a << 1)
#define FIFO_RD_AUX_LSB			(0x5b << 1)

/* These are the state of touch event state machine */
enum {
	EVENT_INIT,
	EVENT_MIDDLE,
	EVENT_RELEASE,
	EVENT_FIFO_END
};

struct max11803_data {
	struct i2c_client		*client;
	struct input_dev		*input_dev;
};

static u8 read_register(struct i2c_client *client, int addr)
{
	/* XXX: The chip ignores LSB of register address */
	return i2c_smbus_read_byte_data(client, addr << 1);
}

static int max11803_write_reg(struct i2c_client *client, int addr, int data)
{
	/* XXX: The chip ignores LSB of register address */
	return i2c_smbus_write_byte_data(client, addr << 1, data);
}

static int max11803_write_cmd(struct i2c_client *client, int command)
{
	return i2c_smbus_write_byte(client, command);
}

static irqreturn_t max11803_ts_interrupt(int irq, void *dev_id)
{
	struct max11803_data *data = dev_id;
	struct i2c_client *client = data->client;
	int status, i, ret;
	u8 buf[XY_BUFSIZE];
	int x = -1;
	int y = -1;

	status = read_register(data->client, GENERAL_STATUS_REG);

	if (status & MAX11803_CONT_INT) {
		/* XY_combined_measurement */
		max11803_write_cmd(client, XY_combined_measurement);
		udelay(4400);

		ret = i2c_smbus_read_i2c_block_data(client,
					FIFO_RD_X_MSB, XY_BUFSIZE, buf);
		if (ret < XY_BUFSIZE)
			goto out;

		for (i = 0; i < XY_BUFSIZE; i += XY_BUFSIZE / 2) {
			if ((buf[i + 1] & MEASURE_TAG_MASK) == MEASURE_X_TAG)
				x = (buf[i] << XY_BUF_OFFSET) +
				    (buf[i + 1] >> XY_BUF_OFFSET);
			else if ((buf[i + 1] & MEASURE_TAG_MASK) == MEASURE_Y_TAG)
				y = (buf[i] << XY_BUF_OFFSET) +
				    (buf[i + 1] >> XY_BUF_OFFSET);
		}

		if (((buf[1] & EVENT_TAG_MASK) != (buf[3] & EVENT_TAG_MASK)) ||
					x < 0 || y < 0)
			goto out;

		switch (buf[1] & EVENT_TAG_MASK) {
		case EVENT_INIT:
			/* fall through */
		case EVENT_MIDDLE:
			input_report_abs(data->input_dev, ABS_X, y); /*X-Y swap*/
			input_report_abs(data->input_dev, ABS_Y, x);
			input_event(data->input_dev, EV_KEY, BTN_TOUCH, 1);
			input_sync(data->input_dev);
			break;

		case EVENT_RELEASE:
			input_event(data->input_dev, EV_KEY, BTN_TOUCH, 0);
			input_sync(data->input_dev);
			break;

		case EVENT_FIFO_END:
			break;
		}
	}
out:
	return IRQ_HANDLED;
}

static void max11803_ts_phy_init(struct max11803_data *data)
{
	struct i2c_client *client = data->client;

	/*
	 * Use continuous interrupt with
	 * direct conversion mode
	 */
	max11803_write_reg(client, GENERAL_CONF_REG, 0xf3);
	/* Average X,Y, take 16 samples, average eight media sample */
	max11803_write_reg(client, MESURE_AVER_CONF_REG, 0xff);
	/* X,Y panel setup time set to 20us */
	max11803_write_reg(client, PANEL_SETUP_TIME_CONF_REG, 0x11);
	/* Rough pullup time (2us), Fine pullup time (500us)  */
	max11803_write_reg(client, TOUCH_DETECT_PULLUP_CONF_REG, 0x99);
	/* Enable Power */
	max11803_write_reg(client, OP_MODE_CONF_REG, 0x06);
}

static int max11803_ts_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct max11803_data *data;
	struct input_dev *input_dev;
	int error;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	input_dev = devm_input_allocate_device(&client->dev);
	if (!data || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	data->client = client;
	data->input_dev = input_dev;

	input_dev->name = "max11803_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_X, 0, MAX11803_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX11803_MAX_Y, 0, 0);
	input_set_drvdata(input_dev, data);

	max11803_ts_phy_init(data);

	error = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					  max11803_ts_interrupt,
					  IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					  "max11803_ts", data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		return error;
	}

	error = input_register_device(data->input_dev);
	if (error)
		return error;

	i2c_set_clientdata(client, data);
	return 0;
}

static const struct i2c_device_id max11803_ts_id[] = {
	{"max11803", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, max11803_ts_id);

static struct i2c_driver max11803_ts_driver = {
	.driver = {
		.name	= "max11803_ts",
	},
	.id_table	= max11803_ts_id,
	.probe		= max11803_ts_probe,
};

module_i2c_driver(max11803_ts_driver);

MODULE_AUTHOR("djeman <djeman@msieurlolo.fr>");
MODULE_DESCRIPTION("Touchscreen driver for MAXI MAX11803 controller");
MODULE_LICENSE("GPL");

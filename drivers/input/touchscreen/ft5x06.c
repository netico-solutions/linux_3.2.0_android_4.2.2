/*
 * Copyright (C) 2016 Nenad Radulovic, <nenad.radulovic@gmail.com>
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
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * This is a driver for the Newhaven "FocalTech" family of touch controllers
 * based on the FocalTech FT5x06 line of chips.
 *
 * Development of this driver has been sponsored by Senis:
 *     http://www.senis.ch
 */

#define DEBUG
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/input/ft5x06.h>

#define MAX_SUPPORT_POINTS		5

#define FT5X06_REG_DEVICE_MODE          0x00
#define FT5X06_REG_GESTURE_ID           0x01
#define FT5X06_REG_TOUCH_POINTS         0x02
#define FT5X06_REG_TOUCH_1_EVENT_FLAG   0x03
#define FT5X06_REG_TOUCH1_XH            0x03
#define FT5X06_REG_TOUCH1_XL            0x04
#define FT5X06_REG_TOUCH1_YH            0x05
#define FT5X06_REG_TOUCH1_YL            0x06

#define FT5X06_REG_ID_G_THGROUP         0x80
#define FT5X06_REG_ID_G_THPEAK          0x81
#define FT5X06_REG_ID_G_THCAL           0x82
#define FT5X06_REG_ID_G_THWATER         0x83
#define FT5X06_REG_ID_G_THTEMP          0x84
#define FT5X06_REG_ID_G_THDIFF          0x85
#define FT5X06_REG_ID_G_CTRL            0x86
#define FT5X06_REG_ID_G_TIME_ENTER_MONITOR 0x87
#define FT5X06_REG_ID_G_PERIODACTIVE    0x88
#define FT5X06_REG_ID_G_PERIODMONITOR   0x89
#define FT5X06_REG_ID_G_AUTO_CLB_MODE   0xa0
#define FT5X06_REG_ID_G_LIB_VERSION_H   0xa1
#define FT5X06_REG_ID_G_LIB_VERSION_L   0xa2
#define FT5X06_REG_ID_G_CIPHER          0xa3
#define FT5X06_REG_ID_G_MODE            0xa4
#define FT5X06_REG_ID_G_PMODE           0xa5
#define FT5X06_REG_ID_G_FIRMID          0xa6
#define FT5X06_REG_ID_G_STATE           0xa7
#define FT5X06_REG_ID_G_FT5201ID        0xa8
#define FT5X06_REG_ID_G_ERR             0xa9

#define FT5X06_TOUCH_EVENT_FLAG_DOWN	0x00
#define FT5X06_TOUCH_EVENT_FLAG_UP		0x01
#define FT5X06_TOUCH_EVENT_ON			0x02
#define FT5X06_TOUCH_EVENT_RESERVED		0x03

#define FT5X06_NAME_LEN			        16

struct ft5x06_data {
	struct i2c_client *     client;
	struct input_dev *      input;

    /* Needed by input subsystem */
	char                    name[FT5X06_NAME_LEN];
};

static int 
ft5x06_readwrite(struct ft5x06_data * tsdata,
        uint16_t wr_len, u8 *wr_buf,
	uint16_t rd_len, u8 *rd_buf)
{
	struct i2c_msg wrmsg[2];
	int i = 0;
	int ret;

	if (wr_len) {
		wrmsg[i].addr  = tsdata->client->addr;
		wrmsg[i].flags = 0;
		wrmsg[i].len = wr_len;
		wrmsg[i].buf = wr_buf;
		i++;
	}

	if (rd_len) {
		wrmsg[i].addr  = tsdata->client->addr;
		wrmsg[i].flags = I2C_M_RD;
		wrmsg[i].len = rd_len;
		wrmsg[i].buf = rd_buf;
		i++;
	}
	ret = i2c_transfer(tsdata->client->adapter, wrmsg, i);

	if (ret < 0)
		return ret;
	if (ret != i)
		return -EIO;

	return 0;
}

static int 
ft5x06_register_write(struct ft5x06_data * tsdata, u8 addr, u8 value)
{
        int error;
	u8 wrbuf[2];

	wrbuf[0] = addr;
	wrbuf[1] = value;

	error = ft5x06_readwrite(tsdata, sizeof(wrbuf), wrbuf, 0, NULL);

        return (error);
}

static int 
ft5x06_register_read(struct ft5x06_data *tsdata, u8 addr, u8 * data)
{
	int error;

	error = ft5x06_readwrite(tsdata, 1, &addr, 1, data);

        return error;
}

#define FT5X06_MODE_NORMAL              0
#define FT5X06_MODE_TEST                4
#define FT5X06_MODE_SYSTEM              1

#define FT5X06_GESTURE_ZOOM_IN          0x48
#define FT5X06_GESTURE_ZOOM_OUT         0X49
#define FT5X06_GESTURE_NONE             0

static void
parse_touch_data(struct ft5x06_data * tsdata, const u8 * rdbuf)
{
        u32     x_data, y_data, point_idx;
        struct device * dev = &tsdata->client->dev;

        for (point_idx = 0; point_idx < MAX_SUPPORT_POINTS; point_idx++) {
                bool down;
                int type;
                const u8 * buf = &rdbuf[point_idx * 6 + 3];

                type = buf[0] >> 6;

                if (type == FT5X06_TOUCH_EVENT_RESERVED)
                        continue;

                x_data = (((u32)buf[0] << 8u) | (u32)buf[1]) & 0xfffu; 
                y_data = (((u32)buf[2] << 8u) | (u32)buf[3]) & 0xfffu; 

                dev_dbg(dev, "t%d: x=%d, y=%d\n", point_idx, x_data, y_data);

                down = type != FT5X06_TOUCH_EVENT_FLAG_UP;
                input_mt_slot(tsdata->input, point_idx);
                input_mt_report_slot_state(tsdata->input, MT_TOOL_FINGER, down);

                if (!down)
                        continue;

                input_report_abs(tsdata->input, ABS_MT_POSITION_X, x_data);
                input_report_abs(tsdata->input, ABS_MT_POSITION_Y, y_data);
        }
        input_mt_report_pointer_emulation(tsdata->input, true);
        input_sync(tsdata->input);
}

static irqreturn_t 
ft5x06_isr(int irq, void *dev_id)
{
	struct ft5x06_data *     tsdata = dev_id;
	u8                          cmd = 0;
	u8                          rdbuf[0x1e];
	int error;

	memset(rdbuf, 0, sizeof(rdbuf));

	error = ft5x06_readwrite(tsdata,
					sizeof(cmd), &cmd,
					sizeof(rdbuf), rdbuf);
	if (error) {
		dev_err(&tsdata->client->dev, "Unable to fetch data, error: %d\n",
				    error);
		goto out;
	}
        parse_touch_data(tsdata, rdbuf);
out:
	return IRQ_HANDLED;
}

static int 
ft5x06_reset(struct ft5x06_data * tsdata, int reset_pin)
{
        dev_dbg(&tsdata->client->dev, "Reseting the device\n");

	if (gpio_is_valid(reset_pin)) {
		gpio_set_value(reset_pin, 0);
		mdelay(50);
		gpio_set_value(reset_pin, 1);
		mdelay(100);
        }

	return 0;
}

struct chip_id {
        u8              id;
        const char *    name;
};

struct chip_id chip_ids[] = {
        {0x55, "nh_ft5x06"},
        {0, NULL}
};

static int 
ft5x06_identify(struct ft5x06_data * tsdata, char * model_name)
{
        struct chip_id * chip_id;
        u8 cipher;
        u8 firmid;
	int error;

        /* Get ID_G_CIPHER : Chip Vendor ID 
         */
	error = ft5x06_register_read(tsdata, FT5X06_REG_ID_G_CIPHER, &cipher);

	if (error)
		return error;

        /* Get ID_G_FIRMID : Firmware ID
         */
        error = ft5x06_register_read(tsdata, FT5X06_REG_ID_G_FIRMID, &firmid);

        if (error)
                return error;

        dev_dbg(&tsdata->client->dev, "Chip Vendor ID %x, Firmware ID %x\n",
                cipher, firmid);

        for (chip_id = chip_ids; (chip_id->id != cipher) && chip_id->name; 
                chip_id++) {
                ;
        }

        if (chip_id->name == NULL)
                return (-ENODEV);

        snprintf(model_name, FT5X06_NAME_LEN, "%s-%02x.%02x", chip_id->name, 
                cipher, firmid);

	return 0;
}

static int ft5x06_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	const struct ft5x06_platform_data *pdata =
						client->dev.platform_data;
	struct ft5x06_data *tsdata;
	struct input_dev * input;
	int error;

	dev_dbg(&client->dev, "probing for FT5x06 I2C\n");

	if (!pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -EINVAL;
	}

	if (gpio_is_valid(pdata->irq_pin)) {
		error = gpio_request_one(pdata->irq_pin, GPIOF_IN, 
                        "ft5x06 irq");

		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d as irq pin, "
                                "error %d\n",
				pdata->irq_pin, error);
			return (error);
		}
	}
	if (gpio_is_valid(pdata->reset_pin)) {
		/* this pulls reset down, enabling the low active reset */
		error = gpio_request_one(pdata->reset_pin, GPIOF_OUT_INIT_LOW, 
                        "ft5x06 reset");

		if (error) {
			dev_err(&client->dev,
			        "Failed to request GPIO %d as reset pin, "
                                "error %d\n",
				pdata->reset_pin, error);
			return error;
		}
        }
	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	input = input_allocate_device();

	if (!tsdata || !input) {
		dev_err(&client->dev, "failed to allocate driver data.\n");
		error = -ENOMEM;
		goto err_free_mem;
	}
	tsdata->client = client;
	tsdata->input = input;
	error = ft5x06_reset(tsdata, pdata->reset_pin);

	if (error) {
		dev_err(&client->dev, "failed to reset the chip\n");
		goto err_free_mem;
    }
	error = ft5x06_identify(tsdata, tsdata->name);

	if (error) {
		dev_err(&client->dev, "touchscreen probe failed\n");
		goto err_free_mem;
	}
	input->name = tsdata->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	input_set_abs_params(input, ABS_X, 0, 800, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, 480, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, 800, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, 480, 0, 0);
	error = input_mt_init_slots(input, MAX_SUPPORT_POINTS);

	if (error) {
		dev_err(&client->dev, "Unable to init MT slots.\n");
		goto err_free_mem;
	}
	input_set_drvdata(input, tsdata);
	i2c_set_clientdata(client, tsdata);

	error = request_threaded_irq(client->irq, NULL, ft5x06_isr,
				     IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				     client->name, tsdata);
	if (error) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		goto err_free_mem;
	}
	error = input_register_device(input);

	if (error)
		goto err_remove_attrs;

	device_init_wakeup(&client->dev, 1);
	dev_dbg(&client->dev,
		"Initialized: IRQ pin %d, Reset pin %d.\n",
		pdata->irq_pin, pdata->reset_pin);

	return 0;

err_remove_attrs:
	free_irq(client->irq, tsdata);
err_free_mem:
	input_free_device(input);
	kfree(tsdata);

	if (gpio_is_valid(pdata->irq_pin))
		gpio_free(pdata->irq_pin);

	return error;
}

static int ft5x06_remove(struct i2c_client *client)
{
	const struct ft5x06_platform_data * pdata =
						dev_get_platdata(&client->dev);
	struct ft5x06_data * tsdata = i2c_get_clientdata(client);

	free_irq(client->irq, tsdata);
	input_unregister_device(tsdata->input);

	if (gpio_is_valid(pdata->irq_pin))
		gpio_free(pdata->irq_pin);
	if (gpio_is_valid(pdata->reset_pin))
		gpio_free(pdata->reset_pin);

	kfree(tsdata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ft5x06_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int ft5x06_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(client->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ft5x06_pm_ops,
			 ft5x06_suspend, ft5x06_resume);

static const struct i2c_device_id ft5x06_id[] = {
	{ "nh_ft5x06", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ft5x06_id);

static struct i2c_driver ft5x06_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "nh_ft5x06",
		.pm = &ft5x06_pm_ops,
	},
	.id_table = ft5x06_id,
	.probe    = ft5x06_probe,
	.remove   = ft5x06_remove,
};

static int __init ft5x06_driver_init(void)
{
    return i2c_add_driver(&ft5x06_driver);
}
module_init(ft5x06_driver_init);

static void __exit ft5x06_driver_exit(void)
{
    i2c_del_driver(&ft5x06_driver);
}
module_exit(ft5x06_driver_exit);

MODULE_AUTHOR("Nenad Radulovic <nenad.radulovic@gmail.com>");
MODULE_DESCRIPTION("Newhaven FT5x06 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");

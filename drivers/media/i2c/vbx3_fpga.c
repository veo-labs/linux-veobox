/*
 * VBX3 FPGA i2c driver.
 * Copyright (C) 2014  Jean-Michel Hautbois
 *
 * A/V source switching Vodalys VBX3
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <linux/i2c.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>

MODULE_DESCRIPTION("i2c device driver for VBX3 fpga source switch");
MODULE_AUTHOR("Jean-Michel Hautbois");
MODULE_LICENSE("GPL");

/* i2c implementation */

static int vbx3_fpga_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	v4l_info(client, "chip found @ 0x%x (%s)\n",
			client->addr << 1, client->adapter->name);

	return 0;
}

static int vbx3_fpga_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	return 0;
}

/* ----------------------------------------------------------------------- */

static const struct i2c_device_id vbx3_fpga_id[] = {
	{ "vbx3_fpga", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, vbx3_fpga_id);

static struct i2c_driver vbx3_fpga_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "vbx3_fpga",
	},
	.probe		= vbx3_fpga_probe,
	.remove		= vbx3_fpga_remove,
	.id_table	= vbx3_fpga_id,
};

module_i2c_driver(vbx3_fpga_driver);

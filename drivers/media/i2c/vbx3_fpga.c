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

#define	VBX3_FPGA_REG_VERSION			0x00
#define	VBX3_FPGA_REG_CTRL_CHAN0		0x01
#define	VBX3_FPGA_REG_CTRL_CHAN1		0x02
#define	VBX3_FPGA_REG_CTRL_PATTERN_CHAN0	0x03
#define	VBX3_FPGA_REG_GLOBAL_STATUS		0x04
#define	VBX3_FPGA_REG_STATUS_SDI0		0x05
#define	VBX3_FPGA_REG_STATUS_SDI1		0x06
#define	VBX3_FPGA_REG_EVENT_CHAN0		0x07
#define	VBX3_FPGA_REG_EVENT_CHAN1		0x08

#define	VBX3_FPGA_SINK_CHAN0	0
#define	VBX3_FPGA_SINK_CHAN1	1
#define	VBX3_FPGA_SOURCE_CHAN0	2
#define	VBX3_FPGA_SOURCE_CHAN1	3

#define	VBX3_FPGA_PADS_NUM	4

enum vbx3_fpga_input_channel0 {
	VBX3_FPGA_INPUT_SDI0,
	VBX3_FPGA_INPUT_ADV7611_HDMI,
};
enum vbx3_fpga_input_channel1 {
	VBX3_FPGA_INPUT_SDI1,
	VBX3_FPGA_INPUT_ADV7604_HDMI,
	VBX3_FPGA_INPUT_ADV7604_VGA,
};

struct vbx3_fpga_state {
	struct v4l2_subdev sd;
	struct media_pad pads[VBX3_FPGA_PADS_NUM];
	enum vbx3_fpga_input_channel0 input_channel0;
	enum vbx3_fpga_input_channel1 input_channel1;
};

static inline struct vbx3_fpga_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct vbx3_fpga_state, sd);
}

static int vbx3_fpga_s_routing (struct v4l2_subdev *sd,
				u32 input, u32 output, u32 config)
{
	struct vbx3_fpga_state *state = to_state(sd);
	return 0;
}

static const struct v4l2_subdev_video_ops vbx3_fpga_video_ops = {
	.s_routing = vbx3_fpga_s_routing,
};

static const struct v4l2_subdev_ops vbx3_fpga_ops = {
	.video = &vbx3_fpga_video_ops,
};


/* i2c implementation */

static int vbx3_fpga_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct vbx3_fpga_state *state;
	int version;
	int ret = 0;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	v4l_info(client, "chip found @ 0x%x (%s)\n",
			client->addr << 1, client->adapter->name);

	version = i2c_smbus_read_byte_data(client, VBX3_FPGA_REG_VERSION);
	if (version < 0) {
		v4l_err(client, "could not get version of FPGA\n");
		return -EPROBE_DEFER;
	} else {
		v4l_info(client, "version read : 0x%x\n", version);
	}

	state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &vbx3_fpga_ops);
	strlcpy(sd->name, "VBX3 FPGA video switch", sizeof(sd->name));

	state->pads[VBX3_FPGA_SINK_CHAN0].flags = MEDIA_PAD_FL_SINK;
	state->pads[VBX3_FPGA_SINK_CHAN1].flags = MEDIA_PAD_FL_SINK;
	state->pads[VBX3_FPGA_SOURCE_CHAN0].flags = MEDIA_PAD_FL_SOURCE;
	state->pads[VBX3_FPGA_SOURCE_CHAN1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_init(&state->sd.entity, VBX3_FPGA_PADS_NUM, state->pads, 0);
	if (ret < 0)
		v4l2_err(client, "media entity failed with error %d\n", ret);

	return ret;
}

static int vbx3_fpga_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct vbx3_fpga_state *state;

	sd = &state->sd;
	media_entity_cleanup(&sd->entity);
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
